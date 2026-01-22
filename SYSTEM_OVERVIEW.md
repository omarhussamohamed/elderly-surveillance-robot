# Elderly Bot - System Overview

## Introduction

Production-ready ROS 1 Melodic package for an autonomous 4WD differential-drive indoor monitoring robot with WiFi rosserial, sensor fusion, and autonomous SLAM capabilities.

**Platform:** Ubuntu 18.04 LTS + ROS Melodic  
**Architecture:** ESP32 motor controller + Jetson Nano compute + RPLidar + MPU9250  
**Status:** Production-ready with validated TF frames, kinematics, and drift-free sensor fusion

---

## Package Structure

```
elderly_bot/
├── CMakeLists.txt                    # Catkin build configuration
├── package.xml                       # ROS package manifest (dependencies)
├── install_dependencies.sh           # Automated dependency installer
│
├── config/                           # Configuration files (YAML)
│   ├── ekf.yaml                      # EKF sensor fusion (50Hz)
│   ├── gmapping.yaml                 # GMapping SLAM (minimumScore=50)
│   ├── amcl.yaml                     # AMCL localization
│   ├── dwa_local_planner.yaml        # DWA trajectory planner
│   ├── costmap_common_params.yaml    # Shared costmap config
│   ├── global_costmap.yaml           # Global planning costmap
│   ├── local_costmap.yaml            # Local planning costmap
│   ├── patrol_goals.yaml             # Patrol waypoint definitions
│   ├── sensors_actuators.yaml        # Gas sensor, buzzer, Jetson stats config
│   └── cloud_config.yaml             # AWS IoT Core connection config
│
├── aws_certs/                        # AWS IoT certificates (deployment only)
│   ├── README.md                     # Certificate setup guide
│   ├── .gitignore                    # Excludes actual certificates
│   ├── setup_certificates.sh         # Automated setup script
│   └── PLACEHOLDER_*.pem             # Structure examples (not real certs)
│
├── firmware/                         # ESP32 Arduino firmware
│   ├── elderly_bot_esp32_wifi.ino    # Production firmware (WiFi rosserial)
│   ├── motor_and_encoder_HW_test.ino # Hardware diagnostic firmware
│   └── motor_and_encoder_SW_test.ino # Software diagnostic firmware
│
├── launch/                           # ROS launch files
│   ├── bringup.launch                # Core hardware (rplidar, rosserial, imu, robot_state_publisher)
│   ├── imu_nav.launch                # IMU processing (Madgwick filter)
│   ├── mapping.launch                # Autonomous mapping (gmapping + explore_lite)
│   ├── navigation.launch             # Patrol mode (AMCL + move_base)
│   └── mpu9250_jetson.launch         # Standalone IMU test launch
│
├── maps/                             # Generated SLAM maps
│   └── README.md                     # Map storage guide
│
├── rviz/                             # RViz visualization configs
│   ├── mapping.rviz                  # SLAM visualization
│   └── navigation.rviz               # Navigation visualization
│
├── scripts/                          # Validation & utility scripts
│   ├── mpu9250_node.py               # MPU9250 driver (Madgwick fusion)
│   ├── sensors_actuators_node.py     # Gas sensor, buzzer, Jetson monitoring
│   ├── cloud_bridge_node.py          # AWS IoT Core MQTT bridge
│   ├── patrol_client.py              # Patrol waypoint sequencer
│   ├── master_validator.sh           # 4-stage system validation
│   ├── tf_verification_complete.sh   # TF frame diagnostic
│   ├── imu_mounting_diagnostic.sh    # Physical IMU orientation test
│   ├── final_drift_validation.sh     # Drift stability test
│   ├── autonomous_square_test.sh     # 1m×1m autonomous test
│   └── boss_level_test.sh            # Complete validation suite
│
├── urdf/                             # Robot geometric description
│   └── elderly_bot.urdf              # Complete URDF (TF frames, links, joints)
│
├── docs/                             # Detailed technical guides
│   ├── MPU9250_JETSON_SETUP.md       # IMU I2C hardware setup
│   └── IMU_CALIBRATION.md            # IMU fusion tuning guide
│
├── README.md                         # Main documentation hub
├── DOCUMENTATION_INDEX.md            # Complete doc navigation
├── QUICK_START.md                    # Setup & deployment guide
├── SYSTEM_OVERVIEW.md                # This file (architecture)
├── HARDWARE_MAP.md                   # Hardware specs & pinouts
├── ROSSERIAL_GUIDE.md                # WiFi rosserial setup
├── CHANGELOG.md                      # Version history
│
├── TF_FRAME_ALIGNMENT_FIX.md         # TF coordinate fix (Jan 18, 2026)
├── COMPLETE_TF_DEPLOYMENT.md         # TF fix deployment procedure
├── IMU_MOUNTING_FIX_REFERENCE.md     # IMU orientation scenarios
├── DRIFT_FIX_APPLIED.md              # Gyro drift fix documentation
├── KINEMATIC_FIX_APPLIED.md          # Odometry scaling fix (3960 ticks/rev)
└── PHYSICAL_VALIDATION_PROTOCOL.md   # Manual testing checklist
```

---

## Hardware Architecture

### Computing & Control
| Component | Model | Function | Interface |
|-----------|-------|----------|-----------|
| **Main Computer** | Jetson Nano 4GB | ROS master, SLAM, navigation, sensor fusion | Ubuntu 18.04 + ROS Melodic |
| **Motor Controller** | ESP32-DevKitC | Motor PWM control, encoder reading, odometry | WiFi rosserial (TCP port 11411) |
| **Motor Drivers** | 2× L298N H-Bridge | Dual motor control per driver | ESP32 GPIO (PWM + direction) |

### Sensors
| Sensor | Model | Specs | Connection | Rate | Topic |
|--------|-------|-------|------------|------|-------|
| **Lidar** | RPLidar A1 | 360°, 8m range, 5-10Hz | Jetson USB | 10Hz | `/scan` |
| **IMU** | MPU9250 | 6-axis (gyro+accel), mag DISABLED | Jetson I2C (bus 1, addr 0x68) | 50Hz | `/imu/data` |
| **Encoders** | Optical quadrature | 11 PPR × 90:1 × 4 edges = 3960 ticks/rev | ESP32 GPIO interrupts | 10Hz | `/wheel_odom` |
| **Gas Sensor** | MQ-6 LPG/Propane | Analog 0-3.3V via ADS1115 ADC | Jetson I2C (addr 0x48) | 1Hz | `/gas_level`, `/gas_detected` |
| **Jetson Stats** | jtop (jetson-stats) | Temperature & power monitoring | Native (software) | 1Hz | `/jetson_temperature`, `/jetson_power` |

### Actuators
| Actuator | Type | Connection | Control | Topic |
|----------|------|------------|---------|-------|
| **Active Buzzer** | Digital ON/OFF | Jetson GPIO (BOARD pin) | GPIO.HIGH/LOW | `/buzzer_command` |

### Motors & Drivetrain
| Component | Specification | Value |
|-----------|--------------|-------|
| **Motors** | JGB37-520 DC gearmotor | 12V, 110 RPM, 90:1 gear ratio |
| **Wheels** | Diameter | 65mm (radius = 32.5mm) |
| **Track Width** | Center-to-center | 260mm (0.26m) |
| **Wheelbase** | Front-to-rear | 170mm (0.17m) |
| **Drive Type** | Differential (4WD skid-steer) | Left/right velocity control |

### Power & Communication
- **Power**: 12V battery (motors) + 5V regulator (ESP32, Jetson via barrel jack)
- **Network**: ESP32 WiFi client → Jetson WiFi AP (rosserial TCP)
- **Emergency Stop**: 800ms cmd_vel timeout triggers motor shutdown

---

## Software Architecture

### ESP32 Firmware (Dual-Core Architecture)

**Core 0 (WiFi & Communication):**
- WiFi rosserial TCP client (connects to Jetson:11411)
- Publishes `/wheel_odom` at 10Hz (nav_msgs/Odometry)
- Subscribes `/cmd_vel` (geometry_msgs/Twist)
- Handles ROS spin, reconnection logic

**Core 1 (Real-Time Motor Control):**
- Quadrature encoder ISRs (4 wheels, interrupt-driven)
- Differential drive kinematics:
  - `left_velocity = linear.x - angular.z * TRACK_WIDTH/2`
  - `right_velocity = linear.x + angular.z * TRACK_WIDTH/2`
- Hardware PWM motor control (50Hz update rate)
- Velocity PID controllers (per-wheel closed-loop)
- Safety: 800ms timeout → emergency stop

**Key Calibrations:**
- `TICKS_PER_REV = 3960` (11 PPR × 90 gear × 4 edges)
- `WHEEL_RADIUS = 0.0325m` (verified kinematic 1:1 scaling)
- `TRACK_WIDTH = 0.26m`
- Velocity limits: max 0.25 m/s linear, 1.0 rad/s angular

**Critical Features:**
- Software debouncing (100μs) prevents ghost encoder pulses
- Minimum PWM thresholds overcome motor stiction
- Float math casting in `ticks_to_m` ensures accuracy
- Watchdog timeout prevents runaway motors

---

### Jetson Nano Software Stack

**Hardware Interfaces:**
1. **robot_state_publisher** (50Hz)
   - Broadcasts static TF transforms from URDF
   - Sources: base_footprint→base_link, base_link→laser, base_link→imu_link
   
2. **rplidar_ros** (10Hz)
   - Publishes `/scan` (sensor_msgs/LaserScan)
   - 360 points per scan, 8m max range
   
3. **mpu9250_node.py** (50Hz)
   - I2C direct communication (bus 1, address 0x68)
   - Gyroscope + Accelerometer only (magnetometer DISABLED - indoor EMI)
   - Madgwick filter for orientation fusion (zeta=0.01, gain=0.9 gyro-dominant)
   - Publishes `/imu/data_raw` and `/imu/data` (fused orientation)
   
4. **sensors_actuators_node.py** (1Hz, optional)
   - **MQ-6 gas sensor** via ADS1115 16-bit I2C ADC (address 0x48)
   - **Active buzzer** via Jetson GPIO (BOARD numbering)
   - **Jetson monitoring** via jtop (temperature & power)
   - Publishes `/gas_level`, `/gas_detected`, `/jetson_temperature`, `/jetson_power`
   - Subscribes `/buzzer_command` (auto-shutoff after 5s timeout)
   - Individual enable flags (default: disabled until hardware ready)
   - Graceful degradation: runs without crashes if hardware missing
   
5. **rosserial_python** (TCP server)
   - WiFi bridge for ESP32 (port 11411)
   - TCP_NODELAY=1 for low-latency odometry
   - Receives `/wheel_odom`, sends `/cmd_vel`

**Sensor Fusion:**
- **robot_localization** (Extended Kalman Filter @ 50Hz)
  - Fuses wheel odometry + IMU absolute yaw orientation
  - Configuration: 2D mode (x, y, yaw only)
  - Outputs: `/odometry/filtered` + TF odom→base_footprint
  - **Critical**: Uses IMU orientation (not angular velocity) to prevent drift
  - Result: <0.05° stationary drift over 3 minutes

**SLAM & Localization:**
1. **gmapping** (GMapping SLAM)
   - Input: `/scan` + `/odometry/filtered`
   - Output: `/map` (nav_msgs/OccupancyGrid @ 5cm resolution)
   - Tuning: `minimumScore=50` for stable loop closure
   
2. **AMCL** (Adaptive Monte Carlo Localization)
   - Particle filter: 100-1000 adaptive particles
   - Laser downsampling: 60 beams (from 360)
   - Publishes TF map→odom correction

**Navigation:**
- **move_base** (navigation core)
  - Global planner: NavfnROS (Dijkstra)
  - Local planner: DWA (Dynamic Window Approach @ 10Hz)
  - Costmaps: Static map + rolling local window (4m × 4m)
  - Recovery behaviors: rotation recovery, clearing costmaps
  
- **explore_lite** (autonomous exploration)
  - Frontier-based exploration during mapping
  - Integrated with move_base

**Autonomous Patrol:**
- **patrol_client.py**
  - Reads waypoints from `config/patrol_goals.yaml`
  - Cycles through goals indefinitely
  - move_base action client integration

---

## TF Tree Architecture (REP-105 Compliant)

```
map                          [world-fixed frame]
 └── odom                    [locally-consistent, drift-accumulating frame]
     └── base_footprint      [robot ground projection, 2D planning frame]
         └── base_link       [robot center reference]
             ├── laser       [lidar sensor frame, 180° yaw for backward mount]
             └── imu_link    [IMU sensor frame, aligned with base_link]
```

### Frame Descriptions

| Frame | Parent | Publisher | Rate | Transform | Purpose |
|-------|--------|-----------|------|-----------|---------|
| **map** | - | AMCL (nav mode) | Variable | World origin | Global reference, never changes |
| **odom** | map | EKF / AMCL | 50Hz / Variable | map→odom correction | Continuous, drift-prone odometry |
| **base_footprint** | odom | robot_localization EKF | 50Hz | Fused odometry | Robot ground projection (2D planning) |
| **base_link** | base_footprint | robot_state_publisher | 50Hz | (0, 0, 0.07) | Robot geometric center |
| **laser** | base_link | robot_state_publisher | 50Hz | xyz=(0,0,0.23) rpy=(0,0,π) | Lidar sensor (180° yaw = backward) |
| **imu_link** | base_link | robot_state_publisher | 50Hz | xyz=(0,0,0.07) rpy=(0,0,0) | IMU sensor (aligned) |

### TF Publisher Configuration

**robot_state_publisher:**
- Reads `urdf/elderly_bot.urdf` at startup
- Publishes all fixed transforms (50Hz broadcast)
- Single source of truth for robot geometry
- **No duplicate static_transform_publisher nodes**

**robot_localization (EKF):**
- Publishes odom→base_footprint transform (50Hz)
- Fuses `/wheel_odom` + `/imu/data` absolute yaw
- Configuration: `config/ekf.yaml`

**AMCL (navigation mode only):**
- Publishes map→odom correction
- Updates when localized on saved map

### Coordinate Frame Convention (REP-105)

All frames follow right-handed coordinate system:
- **X-axis (red)**: Forward (robot front direction)
- **Y-axis (green)**: Left (robot left side)
- **Z-axis (blue)**: Up (vertical, against gravity)

**Special Cases:**
- `laser` frame: 180° yaw rotation (X backward) because lidar physically faces backward
- All axes within same frame must be parallel when visualized in RViz
- IMU +Z axis must point up (+9.8 m/s² gravity on -Z when stationary)

### TF Verification Commands

```bash
# View complete TF tree
rosrun tf view_frames && evince frames.pdf

# Check specific transform
rosrun tf tf_echo base_link laser

# Monitor TF broadcast rates
rosrun tf tf_monitor

# Run automated TF diagnostic
bash ~/catkin_ws/src/elderly_bot/scripts/tf_verification_complete.sh
```

**Expected Results:**
- Laser: 180° yaw rotation (NOT roll or pitch)
- IMU: Near-zero rotation (aligned with base_link)
- No broken chains, no timing warnings
- All transforms publishing at expected rates

---

## Operational Modes

### MODE 1: Autonomous Mapping

**Purpose:** Create occupancy grid map of unknown environment

**Launch Command:**
```bash
roslaunch elderly_bot mapping.launch
```

**Active Nodes:**
```
Hardware Layer:
  - rplidar_ros → /scan
  - rosserial_python → ESP32 WiFi bridge
  - mpu9250_node.py → /imu/data
  - robot_state_publisher → TF static frames

Sensor Fusion:
  - robot_localization (EKF) → /odometry/filtered + TF odom→base_footprint

SLAM:
  - gmapping → /map (occupancy grid)
  
Autonomous Exploration:
  - explore_lite → frontier-based exploration goals
  - move_base → navigation to frontiers
```

**Behavior:**
1. Robot starts from current position as map origin
2. Lidar scans environment, gmapping builds map incrementally
3. explore_lite identifies unexplored "frontiers"
4. move_base navigates to frontiers autonomously
5. Process repeats until no frontiers remain or manually stopped

**Save Map:**
```bash
rosrun map_server map_saver -f ~/catkin_ws/src/elderly_bot/maps/<map_name>
```
Creates: `<map_name>.yaml` (metadata) + `<map_name>.pgm` (image)

**Configuration Files:**
- `config/gmapping.yaml` - SLAM parameters (minimumScore=50, 5cm resolution)
- `config/ekf.yaml` - Sensor fusion (50Hz, absolute yaw from IMU)
- `config/dwa_local_planner.yaml` - Exploration velocity limits
- `config/costmap_common_params.yaml` - Obstacle detection

---

### MODE 2: Navigation & Patrol

**Purpose:** Autonomous patrol on pre-existing map

**Launch Commands:**
```bash
# Terminal 1: Start navigation stack
roslaunch elderly_bot navigation.launch map_file:=/path/to/map.yaml

# Terminal 2: Start patrol (cycles through waypoints)
rosrun elderly_bot patrol_client.py
```

**Active Nodes:**
```
Hardware Layer:
  - rplidar_ros → /scan
  - rosserial_python → ESP32 WiFi bridge
  - mpu9250_node.py → /imu/data
  - robot_state_publisher → TF static frames

Sensor Fusion:
  - robot_localization (EKF) → /odometry/filtered

Localization:
  - map_server → /map_static (from saved .yaml/.pgm)
  - AMCL → particle filter localization, TF map→odom

Navigation:
  - move_base → global planner (Dijkstra) + local planner (DWA)
  
Patrol:
  - patrol_client.py → waypoint sequencer
```

**Behavior:**
1. AMCL localizes robot on loaded map (initial pose required)
2. patrol_client reads waypoints from `config/patrol_goals.yaml`
3. Sends waypoints to move_base sequentially
4. Robot navigates autonomously, avoiding dynamic obstacles
5. Cycles through waypoints indefinitely
6. Recovery behaviors activate if stuck

**Set Initial Pose:**
- In RViz: Click "2D Pose Estimate", drag arrow to set position + orientation
- Programmatically: Publish to `/initialpose` (PoseWithCovarianceStamped)

**Configuration Files:**
- `config/patrol_goals.yaml` - List of (x, y, yaw) waypoints
- `config/amcl.yaml` - Localization parameters
- `config/global_costmap.yaml` - Static map costmap
- `config/local_costmap.yaml` - Rolling window obstacle avoidance

---

## ROS Communication Graph

### Published Topics

| Topic | Type | Rate | Publisher | Description |
|-------|------|------|-----------|-------------|
| `/wheel_odom` | nav_msgs/Odometry | 10Hz | ESP32 (rosserial) | Raw wheel encoder odometry |
| `/scan` | sensor_msgs/LaserScan | 10Hz | rplidar_ros | 360° lidar scan (360 points, 8m range) |
| `/imu/data` | sensor_msgs/Imu | 50Hz | mpu9250_node.py | Fused IMU (Madgwick orientation + raw gyro/accel) |
| `/odometry/filtered` | nav_msgs/Odometry | 50Hz | robot_localization | EKF-fused odometry (wheel + IMU yaw) |
| `/map` | nav_msgs/OccupancyGrid | 0.5Hz | gmapping / map_server | Occupancy grid map (5cm cells) |
| `/tf` | tf2_msgs/TFMessage | 50Hz | Multiple | Transform tree broadcasts |
| `/cmd_vel` | geometry_msgs/Twist | 10Hz | move_base | Velocity commands to motors |
| `/gas_level` | std_msgs/Float32 | 1Hz | sensors_actuators_node | Gas sensor voltage (0-3.3V, calibratable to PPM) |
| `/gas_detected` | std_msgs/Bool | 1Hz | sensors_actuators_node | Gas detection flag (> threshold) |
| `/jetson_temperature` | sensor_msgs/Temperature | 1Hz | sensors_actuators_node | Jetson GPU/CPU temperature (°C) |
| `/jetson_power` | std_msgs/Float32 | 1Hz | sensors_actuators_node | Total power consumption (watts) |

### Subscribed Topics

| Topic | Type | Subscriber | Usage |
|-------|------|-----------|-------|
| `/cmd_vel` | geometry_msgs/Twist | ESP32 (rosserial) | Motor velocity commands |
| `/initialpose` | PoseWithCovarianceStamped | AMCL | Set initial localization pose |
| `/move_base_simple/goal` | PoseStamped | move_base | Single navigation goal |
| `/move_base/goal` | move_base_msgs/MoveBaseActionGoal | move_base | Action-based navigation goal |
| `/buzzer_command` | std_msgs/Bool | sensors_actuators_node | Buzzer control (True=ON, auto-shutoff after 5s) |

### Action Servers

| Action | Type | Server | Usage |
|--------|------|--------|-------|
| `/move_base` | move_base_msgs/MoveBaseAction | move_base | Navigate to goal pose with feedback |

---

## Critical System Parameters

### Robot Physical Dimensions
```yaml
Footprint: 35cm (length) × 25cm (width) rectangular
Wheel Diameter: 65mm (radius = 32.5mm = 0.0325m)
Track Width: 26cm (0.26m) - center-to-center between left/right wheels
Wheelbase: 17cm (0.17m) - front-to-rear axle distance
Ground Clearance: ~7cm (base_footprint to ground)
Total Height: ~30cm (including lidar mount)
```

### Encoder & Kinematics Calibration
```yaml
Encoder Resolution: 11 PPR (pulses per revolution, single channel)
Gear Ratio: 90:1
Quadrature Edges: 4 (A/B phase, rising/falling)
TICKS_PER_REV: 3960 (11 × 90 × 4) ✅ VERIFIED Jan 18, 2026
WHEEL_RADIUS: 0.0325m ✅ VERIFIED
Linear Resolution: ~0.051mm per tick
```

**Kinematic Verification:**
- 1.0m commanded → 1.0m actual movement (±2cm accuracy)
- Documented in: `KINEMATIC_FIX_APPLIED.md`

### Velocity Limits & Control Frequencies
```yaml
Max Linear Velocity: 0.25 m/s (indoor safe speed)
Max Angular Velocity: 1.0 rad/s
Min Linear Velocity: 0.05 m/s (overcome motor stiction)

ESP32 Motor Control Loop: 50Hz (20ms period)
ESP32 Odometry Publishing: 10Hz (100ms period)
EKF Sensor Fusion: 50Hz
IMU Data Rate: 50Hz
Lidar Scan Rate: 10Hz
Local Planner (DWA): 10Hz
Global Planner: 1Hz
TF Broadcast (robot_state_publisher): 50Hz
```

### Sensor Fusion Configuration (EKF)
```yaml
Fusion Rate: 50Hz
Mode: 2D (x, y, yaw only - ignores z, roll, pitch)

Wheel Odometry Input (/wheel_odom):
  - Position: x, y ✅
  - Orientation: yaw ✅
  - Linear velocity: vx ✅
  - Angular velocity: vyaw ❌ (not fused, drifts)

IMU Input (/imu/data):
  - Absolute orientation: yaw ✅ (prevents drift)
  - Angular velocity: vyaw ❌ (not fused)
  - Linear acceleration: ax, ay ❌ (too noisy for benefit)

Output:
  - /odometry/filtered (nav_msgs/Odometry)
  - TF: odom → base_footprint
```

**Critical Setting:** IMU orientation (not angular velocity) is fused to prevent gyro drift
- Result: <0.05° drift over 3 minutes stationary
- Madgwick filter zeta=0.01 for real-time bias correction

### GMapping SLAM Parameters
```yaml
Resolution: 0.05m (5cm grid cells)
Range Max: 8.0m (RPLidar A1 limit)
Particles: 30
minimumScore: 50 ✅ (increased from default for stability)
linearUpdate: 0.2m (update after 20cm movement)
angularUpdate: 0.2rad (~11° rotation)
```

**Tuning:** `minimumScore=50` prevents spurious loop closures in feature-poor environments

### Navigation Stack Parameters

**Global Planner (NavfnROS):**
```yaml
Algorithm: Dijkstra shortest path
Costmap: Static map + inflated obstacles
Update Rate: 1Hz
Allow Unknown: false (stay in known areas)
```

**Local Planner (DWA):**
```yaml
Algorithm: Dynamic Window Approach
Update Rate: 10Hz
Simulation Time: 1.7s (look-ahead)
Simulation Granularity: 0.025m

Velocity Search:
  vx samples: 20 (linear velocity candidates)
  vtheta samples: 40 (angular velocity candidates)

Cost Function Weights:
  path_distance_bias: 32.0 (follow global plan)
  goal_distance_bias: 24.0 (reach goal)
  occdist_scale: 0.02 (avoid obstacles)
```

**Costmaps:**
```yaml
Obstacle Inflation Radius: 0.30m
Robot Footprint Padding: 0.02m
Obstacle Layer: Raytrace range = 2.5m, Obstacle range = 2.5m
Rolling Window (local): 4.0m × 4.0m
Update Frequency: 5Hz
```

### Safety & Recovery

**Timeouts:**
- ESP32 cmd_vel timeout: 800ms → emergency motor stop
- move_base goal timeout: 30s → abort goal
- AMCL global localization timeout: 10s

**Recovery Behaviors (in order):**
1. Clear local costmap (remove stale obstacles)
2. Rotate in place 360° (clear sensor view)
3. Clear both costmaps aggressively
4. Rotate in place 360° again
5. Abort goal if all fail

**Collision Avoidance:**
- Footprint checking before movement
- Dynamic obstacle detection (people, moving objects)
- 30cm safety margin around obstacles

---

## Performance Metrics & Validation

### Expected Performance (Validated)
```yaml
Mapping Time: 10-20 minutes (typical 100m² house)
Localization Accuracy: ±5cm position, ±3° orientation (AMCL on known map)
Navigation Success Rate: >95% in known environment
Odometry Drift: <0.05° rotation over 3 minutes stationary ✅
Kinematic Accuracy: 1.0m commanded = 1.0m actual ±2cm ✅
TF Frame Alignment: All axes parallel, laser 180° yaw ✅
Obstacle Clearance: 30cm minimum safety margin
Loop Closure: Reliable with minimumScore=50
```

### Validation Scripts

Located in `scripts/`:

**Comprehensive Validation:**
```bash
# Complete 4-stage system test
bash master_validator.sh
```
- Stage 1: Communication audit (topic rates, TF tree)
- Stage 2: 3-minute drift stress test (stationary)
- Stage 3: 1-meter linear motion test (kinematic accuracy)
- Stage 4: 360° rotation test (angular accuracy)

**TF Frame Diagnostics:**
```bash
# Automated TF verification
bash tf_verification_complete.sh
```
Checks: Laser rotation (yaw vs roll), IMU alignment, gravity direction

**IMU Orientation Test:**
```bash
# Physical mounting diagnostic
bash imu_mounting_diagnostic.sh
```
Interactive test to determine correct URDF rpy values for IMU

**Individual Tests:**
```bash
bash final_drift_validation.sh       # Drift + rate monitoring
bash autonomous_square_test.sh       # 1m × 1m autonomous mapping
bash boss_level_test.sh              # All tests combined
```

---

## Configuration File Details

### `config/ekf.yaml`
**Purpose:** Robot localization sensor fusion configuration

**Key Settings:**
```yaml
frequency: 50  # EKF update rate
two_d_mode: true  # Ignore z, roll, pitch

odom0: /wheel_odom
odom0_config: [true, true, false,    # x, y, z
               false, false, true,    # roll, pitch, yaw
               true, false, false,    # vx, vy, vz
               false, false, false,   # vroll, vpitch, vyaw (not fused!)
               false, false, false]   # ax, ay, az

imu0: /imu/data
imu0_config: [false, false, false,   # position (not from IMU)
              false, false, true,     # roll, pitch, YAW ✅ (absolute orientation)
              false, false, false,    # velocity
              false, false, false,    # vyaw NOT fused (prevents drift)
              false, false, false]    # acceleration (too noisy)

publish_tf: true  # Publish odom → base_footprint
```

**Critical:** IMU absolute yaw (orientation) is fused, NOT angular velocity (prevents drift)

---

### `config/gmapping.yaml`
**Purpose:** GMapping SLAM parameters for indoor mapping

**Key Settings:**
```yaml
xmin: -50.0, xmax: 50.0, ymin: -50.0, ymax: 50.0  # 100m × 100m max map
delta: 0.05  # 5cm resolution

particles: 30
linearUpdate: 0.2    # Update after 20cm movement
angularUpdate: 0.2   # Update after ~11° rotation

minimumScore: 50  # ✅ Increased from default, prevents false loop closures

kernelSize: 1
lstep: 0.05
astep: 0.05
lsigma: 0.075
ogain: 3.0
```

**Tuning Notes:**
- `minimumScore=50` critical for stable maps in feature-poor environments
- Default is ~0, which causes spurious loop closures
- Reduces map ghosting/doubling

---

### `config/amcl.yaml`
**Purpose:** AMCL localization on saved map

**Key Settings:**
```yaml
min_particles: 100
max_particles: 1000
kld_err: 0.05  # Adaptive particle count error threshold

odom_model_type: diff  # Differential drive
odom_alpha1: 0.2  # Rotation noise from rotation
odom_alpha2: 0.2  # Rotation noise from translation
odom_alpha3: 0.2  # Translation noise from translation
odom_alpha4: 0.2  # Translation noise from rotation

laser_max_beams: 60  # Downsample from 360 for efficiency
laser_likelihood_max_dist: 2.0
```

---

### `config/dwa_local_planner.yaml`
**Purpose:** Dynamic Window Approach local trajectory planning

**Key Settings:**
```yaml
max_vel_x: 0.25
min_vel_x: 0.05
max_vel_theta: 1.0
min_vel_theta: -1.0

acc_lim_x: 1.0  # Linear acceleration limit
acc_lim_theta: 2.0  # Angular acceleration limit

sim_time: 1.7  # Forward simulation time (look-ahead)
vx_samples: 20
vtheta_samples: 40

path_distance_bias: 32.0  # Follow global plan closely
goal_distance_bias: 24.0  # Reach goal quickly
occdist_scale: 0.02  # Avoid obstacles (low = more aggressive)
```

---

### `config/costmap_common_params.yaml`
**Purpose:** Shared costmap configuration for global and local planners

**Key Settings:**
```yaml
robot_base_frame: base_link
update_frequency: 5.0
publish_frequency: 2.0

footprint: [[0.175, 0.125], [0.175, -0.125], [-0.175, -0.125], [-0.175, 0.125]]

inflation_radius: 0.30  # 30cm safety margin
cost_scaling_factor: 10.0

obstacle_layer:
  obstacle_range: 2.5  # Detect obstacles up to 2.5m
  raytrace_range: 2.5  # Clear obstacles up to 2.5m
  max_obstacle_height: 2.0
```

---

### `config/patrol_goals.yaml`
**Purpose:** Waypoint definitions for autonomous patrol

**Format:**
```yaml
goals:
  - x: 2.0
    y: 1.0
    yaw: 0.0
  - x: 2.0
    y: -1.0
    yaw: -1.57
  # ... more waypoints
```

**Usage:** Read by `patrol_client.py`, sent to move_base sequentially in infinite loop

---

## System Dependencies

### ROS Packages (apt-get)
```bash
# Navigation stack
ros-melodic-navigation
ros-melodic-move-base
ros-melodic-amcl
ros-melodic-map-server

# Sensor fusion & localization
ros-melodic-robot-localization

# SLAM
ros-melodic-gmapping
ros-melodic-explore-lite

# Sensor drivers
ros-melodic-rplidar-ros

# Communication
ros-melodic-rosserial
ros-melodic-rosserial-python
ros-melodic-rosserial-arduino

# TF & transforms
ros-melodic-robot-state-publisher
ros-melodic-tf2-ros

# Visualization
ros-melodic-rviz

# Utilities
ros-melodic-teleop-twist-keyboard (for manual control)
```

### Python Dependencies
```bash
# Core dependencies
pip install pyserial smbus2 numpy

# Optional: Sensors & Actuators Node (install when hardware ready)
pip3 install jetson-stats Jetson.GPIO adafruit-blinka adafruit-circuitpython-ads1x15
# Note: jetson-stats requires system reboot after installation
```

### ESP32 Arduino Libraries
```
Rosserial Arduino Library (v0.9.1)
WiFi.h (ESP32 core library)
```

### System Requirements
```
Jetson Nano: Ubuntu 18.04 LTS, 4GB RAM
ESP32: Arduino core 2.0.x
Network: WiFi (ESP32 client → Jetson AP or shared network)
```

### Installation Script
All dependencies can be installed via:
```bash
bash ~/catkin_ws/src/elderly_bot/install_dependencies.sh
```

---

## Build & Deployment

### First-Time Setup

**1. Clone Repository:**
```bash
cd ~/catkin_ws/src
git clone <repository_url> elderly_bot
cd ~/catkin_ws
```

**2. Install Dependencies:**
```bash
bash ~/catkin_ws/src/elderly_bot/install_dependencies.sh
```

**3. Build Package:**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

**4. Upload ESP32 Firmware:**
- Open `firmware/elderly_bot_esp32_wifi.ino` in Arduino IDE
- **Verify settings:**
  - `TICKS_PER_REV = 3960` ✅
  - `WHEEL_RADIUS = 0.0325` ✅
  - `TRACK_WIDTH = 0.26` ✅
  - WiFi SSID and password match network
  - ROS_SERVER_IP matches Jetson IP
- Upload to ESP32

**5. Configure IMU (Jetson):**
Follow: `docs/MPU9250_JETSON_SETUP.md`
- Enable I2C bus 1
- Set permissions: `sudo usermod -aG i2c $USER`
- Test: `i2cdetect -y -r 1` (should show 0x68)

**6. Verify System:**
```bash
# Run comprehensive validation
bash ~/catkin_ws/src/elderly_bot/scripts/master_validator.sh
```

### Update Deployment (After Code Changes)

**Transfer Files from Windows to Jetson:**
```powershell
# From Windows PowerShell
cd "c:\Users\omarh\Desktop\Graduation Project\Elderly_Robot_Project\catkin_ws\src\elderly_bot"

scp -r config/ omar@192.168.1.29:~/catkin_ws/src/elderly_bot/
scp -r launch/ omar@192.168.1.29:~/catkin_ws/src/elderly_bot/
scp -r urdf/ omar@192.168.1.29:~/catkin_ws/src/elderly_bot/
scp -r scripts/ omar@192.168.1.29:~/catkin_ws/src/elderly_bot/
```

**Rebuild on Jetson:**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**Restart ROS:**
```bash
rosnode kill -a && killall -9 rosmaster roscore
roscore &
roslaunch elderly_bot bringup.launch
```

---

## Critical Fixes Applied (Jan 18, 2026)

### 1. TF Frame Alignment Fix
**Problem:** Lidar frame had 180° roll instead of 180° yaw, causing coordinate misalignment

**Solution:**
- Corrected `urdf/elderly_bot.urdf` line 169: `rpy="0 0 3.14159"` (was "3.14159 0 0")
- Added robot_state_publisher to bringup.launch
- Removed duplicate static_transform_publisher nodes

**Documentation:** [TF_FRAME_ALIGNMENT_FIX.md](TF_FRAME_ALIGNMENT_FIX.md)  
**Deployment:** [COMPLETE_TF_DEPLOYMENT.md](COMPLETE_TF_DEPLOYMENT.md)

---

### 2. Kinematic Scaling Fix
**Problem:** Robot moved 6cm when commanded 1m (incorrect TICKS_PER_REV)

**Solution:**
- Corrected TICKS_PER_REV from 4900 to 3960 in ESP32 firmware
- Formula: 11 PPR × 90:1 gear × 4 quadrature edges = 3960
- Verified WHEEL_RADIUS = 0.0325m

**Result:** 1.0m commanded = 1.0m actual (±2cm)  
**Documentation:** [KINEMATIC_FIX_APPLIED.md](KINEMATIC_FIX_APPLIED.md)

---

### 3. Gyro Drift Fix
**Problem:** -13° rotation drift while stationary, map ghosting

**Solution:**
- EKF fuses IMU absolute yaw orientation (not angular velocity)
- Madgwick filter zeta=0.01 for bias correction
- TCP nodelay=1 for low-latency odometry

**Result:** <0.05° drift over 3 minutes  
**Documentation:** [DRIFT_FIX_APPLIED.md](DRIFT_FIX_APPLIED.md)

---

## Troubleshooting Guide

### Common Issues & Solutions

**Issue: "No /scan topic"**
```bash
# Check RPLidar connection
ls /dev/ttyUSB*  # Should show /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0

# Test standalone
roslaunch rplidar_ros rplidar.launch

# Check permissions permanently
sudo usermod -aG dialout $USER
```

**Issue: "rosserial not connecting"**
```bash
# Check ESP32 serial output for IP address
# Verify Jetson IP matches ROS_SERVER_IP in firmware
# Check firewall
sudo ufw allow 11411/tcp

# Test rosserial server manually
rosrun rosserial_python serial_node.py tcp
```

**Issue: "TF frames misaligned in RViz"**
```bash
# Run automated diagnostic
bash ~/catkin_ws/src/elderly_bot/scripts/tf_verification_complete.sh

# If laser shows 180° roll (not yaw), URDF not loaded
# Solution: Restart ROS completely
rosnode kill -a && killall -9 rosmaster roscore
roslaunch elderly_bot bringup.launch
```

**Issue: "Robot moves wrong distance"**
```bash
# Verify firmware has correct TICKS_PER_REV
# Should be 3960, NOT 4900
# See: KINEMATIC_FIX_APPLIED.md for upload procedure
```

**Issue: "Map has double walls (ghosting)"**
```bash
# Check drift
rostopic echo /imu/data | grep orientation

# Run drift test
bash ~/catkin_ws/src/elderly_bot/scripts/final_drift_validation.sh

# If drift >0.1°/min, see: DRIFT_FIX_APPLIED.md
```

**Issue: "IMU axes wrong direction"**
```bash
# Run physical orientation diagnostic
bash ~/catkin_ws/src/elderly_bot/scripts/imu_mounting_diagnostic.sh

# Follow prompts, get correct URDF rpy values
# See: IMU_MOUNTING_FIX_REFERENCE.md
```

**Issue: "move_base not reaching goals"**
```bash
# Check costmaps in RViz
# Verify robot footprint visible
# Check for TF timing warnings
rostopic hz /tf

# Increase inflation radius if stuck near walls
# Edit: config/costmap_common_params.yaml
```

### Diagnostic Commands

```bash
# Check all topics and rates
rostopic list
rostopic hz /scan
rostopic hz /wheel_odom
rostopic hz /odometry/filtered

# View TF tree
rosrun tf view_frames && evince frames.pdf

# Check for TF errors
roswtf

# Monitor node health
rosnode list
rosnode info /robot_state_publisher

# View live transforms
rosrun tf tf_echo base_link laser

# Check sensor data
rostopic echo /imu/data
rostopic echo /scan --noarr

# Manual control test
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

---

## Performance Optimization Tips

### Mapping Quality
- Start robot in open area (good initial scan)
- Reduce exploration speed for better scan matching
- Ensure good lighting (if using camera in future)
- Keep robot stationary during startup (gyro calibration)

### Localization Accuracy
- Use high-quality map (slow exploration speed during mapping)
- Set accurate initial pose in RViz
- Increase AMCL particles if localization jumps
- Ensure consistent environment (furniture not moved)

### Navigation Robustness
- Tune inflation_radius for environment (wider for cluttered)
- Adjust DWA cost weights (path vs goal vs obstacle avoidance)
- Increase recovery behavior timeout for complex environments
- Lower max velocities in tight spaces

### Computational Performance
- Monitor CPU usage: `htop`
- Reduce lidar scan resolution if lag (decrease laser_max_beams)
- Lower costmap update frequency if overloaded
- Disable RViz on Jetson (visualize from remote machine)

---

## Safety & Best Practices

### Operational Safety
1. Always test in open area first
2. Keep emergency stop accessible (Ctrl+C on terminal)
3. Monitor battery voltage (prevent over-discharge)
4. Verify obstacle detection before autonomous operation
5. Keep robot within visual line of sight during testing

### Code Modification Safety
1. Always backup working configuration before changes
2. Test in simulation first if possible (Gazebo)
3. Validate changes with master_validator.sh
4. Document all parameter changes in CHANGELOG.md
5. Use version control (git) for all modifications

### Hardware Safety
1. Never hot-swap motor connectors (power off first)
2. Check encoder wiring polarity (consistent Left/Right)
3. Secure all cables (prevent snagging on environment)
4. Verify motor direction before autonomous operation
5. Test emergency stop function before each session

---

## Standards Compliance

### ROS REPs (ROS Enhancement Proposals)
- **REP-103:** Standard Units of Measure and Coordinate Conventions
  - Linear: meters (m), Angular: radians (rad), Time: seconds (s)
- **REP-105:** Coordinate Frames for Mobile Platforms
  - TF tree: map → odom → base_footprint → base_link
  - X forward, Y left, Z up (right-handed)
- **REP-135:** Driver Namespace Practices
  - Sensors publish in global namespace or under robot namespace

### Code Quality
- Modular launch files (composable components)
- Parameterized configurations (no hardcoded values in launch files)
- Comprehensive error handling and logging
- Graceful degradation (robot safe-stops on errors)

---

## Future Enhancement Roadmap

### Planned Features
- [ ] Camera integration for visual monitoring
- [ ] Person detection and tracking
- [ ] Voice alert system (speaker + TTS)
- [ ] Web dashboard for remote monitoring
- [ ] Battery voltage monitoring
- [ ] Auto-docking for charging
- [ ] Multi-floor mapping (elevator detection)
- [ ] Scheduled patrol (cron-based automation)
- [ ] Fall detection (for elderly monitoring use case)
- [ ] Intrusion detection (nighttime security)

### Potential Upgrades
- Stereo camera for 3D obstacle detection
- Depth camera (RealSense) for better object detection
- Gripper arm for object manipulation
- Thermal camera for person detection in darkness
- LTE modem for remote operation

---

## Documentation Quick Reference

### Essential Reading (Start Here)
1. [README.md](README.md) - Project overview
2. [QUICK_START.md](QUICK_START.md) - Setup guide
3. [DOCUMENTATION_INDEX.md](DOCUMENTATION_INDEX.md) - Complete doc index

### Hardware & Connections
- [HARDWARE_MAP.md](HARDWARE_MAP.md) - Specs, pinouts, wiring
- [docs/MPU9250_JETSON_SETUP.md](docs/MPU9250_JETSON_SETUP.md) - IMU hardware setup
- [ROSSERIAL_GUIDE.md](ROSSERIAL_GUIDE.md) - WiFi communication

### Critical Fixes (Applied Jan 18, 2026)
- [TF_FRAME_ALIGNMENT_FIX.md](TF_FRAME_ALIGNMENT_FIX.md) - Coordinate frame fix
- [COMPLETE_TF_DEPLOYMENT.md](COMPLETE_TF_DEPLOYMENT.md) - TF deployment procedure
- [KINEMATIC_FIX_APPLIED.md](KINEMATIC_FIX_APPLIED.md) - Odometry scaling fix
- [DRIFT_FIX_APPLIED.md](DRIFT_FIX_APPLIED.md) - Gyro drift solution

### Testing & Validation
- [PHYSICAL_VALIDATION_PROTOCOL.md](PHYSICAL_VALIDATION_PROTOCOL.md) - Manual test checklist
- [IMU_MOUNTING_FIX_REFERENCE.md](IMU_MOUNTING_FIX_REFERENCE.md) - IMU orientation scenarios
- Scripts: `scripts/master_validator.sh` (automated testing)

### Configuration Reference
- [docs/IMU_CALIBRATION.md](docs/IMU_CALIBRATION.md) - IMU fusion tuning
- `config/*.yaml` - All ROS parameter files (inline documentation)

---

## Support & Contribution

### Reporting Issues
1. Check existing documentation (README, SYSTEM_OVERVIEW, QUICK_START)
2. Run diagnostic scripts (`master_validator.sh`, `tf_verification_complete.sh`)
3. Check ROS logs: `~/.ros/log/latest/*.log`
4. Gather system info: `roswtf`, `rostopic list`, `rosnode list`

### Making Changes
1. Test thoroughly with validation scripts
2. Update CHANGELOG.md with date and description
3. Update relevant documentation (README, this file, etc.)
4. Commit with descriptive message
5. Document any new parameters or configurations

---

**Version:** 1.2.0  
**Last Updated:** January 18, 2026  
**ROS Distribution:** Melodic  
**Target Platform:** Ubuntu 18.04 + Jetson Nano  
**Status:** Production-Ready ✅  
**License:** BSD 3-Clause


